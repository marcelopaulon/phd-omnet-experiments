package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n100_u16_a8000m2_7
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor;p=475.72,7826.79");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1208.52,7996.86");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1631.76,7872.55");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2383.77,7768.65");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3053.50,7978.85");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4036.85,7917.30");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4673.18,7869.88");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=174.86,7617.53");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1192.87,7639.95");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2809.28,7539.15");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3186.11,7545.20");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3693.60,7646.06");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4120.68,7515.29");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5299.05,7689.01");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=426.73,7348.86");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=882.50,7365.34");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1430.88,7349.86");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1979.53,7433.47");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3833.51,7279.27");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4468.48,7354.44");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5505.19,7298.72");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5876.51,7453.34");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=640.07,7012.51");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=995.21,7015.39");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2475.80,7135.87");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4318.07,7025.36");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4957.68,7175.64");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5797.37,7103.08");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=96.43,6924.28");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1570.80,6840.01");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1979.24,6974.65");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3033.97,6878.05");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3491.97,6992.29");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=99.77,6545.82");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=897.55,6663.99");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1276.23,6611.77");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1949.38,6556.29");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2617.33,6700.14");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4060.57,6742.08");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4463.88,6705.51");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5402.43,6693.07");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5812.22,6593.01");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=478.95,6479.79");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3020.85,6402.61");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3420.03,6317.21");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3791.65,6428.83");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4863.60,6430.81");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1230.96,6251.74");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1899.46,6127.07");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2339.98,6180.53");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4327.34,6217.98");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5387.93,6065.90");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5973.84,6092.12");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=205.24,6007.00");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1132.01,5864.54");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1587.66,5848.18");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2537.50,5864.22");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3063.27,5970.48");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3829.63,5813.44");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=661.05,5560.92");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2074.54,5707.07");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2804.31,5635.94");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3507.45,5570.88");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4775.45,5761.66");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2060.91,5287.19");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2481.55,5474.89");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3169.30,5456.09");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5134.48,5295.17");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1281.84,5265.04");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3004.38,5099.79");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3501.46,5050.86");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3936.62,5252.19");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4413.54,5228.94");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=715.90,5024.29");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1746.59,4953.15");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2493.76,4955.69");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5602.36,4797.89");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=956.10,4662.41");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1523.25,4677.95");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2223.82,4569.45");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2631.51,4626.96");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3120.95,4723.82");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3837.02,4732.89");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4236.91,4600.41");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4783.51,4782.58");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5128.23,4624.60");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=473.96,4527.90");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1388.28,4323.12");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1813.62,4475.86");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3414.53,4336.27");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4662.97,4404.75");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5801.81,4313.54");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=248.71,4257.65");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=795.93,4181.44");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2804.46,4183.17");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4099.56,4255.89");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5340.34,4233.58");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=98.76,3843.94");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=522.69,3873.97");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1455.99,3811.73");
    }

}

