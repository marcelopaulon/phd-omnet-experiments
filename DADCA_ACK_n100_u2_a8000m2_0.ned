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

network DADCA_ACK_n100_u2_a8000m2_0
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
        @display("i=misc/sensor;p=57.67,7852.22");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=938.09,7961.16");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2272.21,7999.53");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3809.11,7810.54");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4513.44,7946.02");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=605.39,7637.27");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1303.29,7729.43");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2751.73,7705.88");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3287.79,7738.53");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4944.36,7553.27");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5371.35,7635.23");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=226.87,7282.17");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=789.13,7303.01");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2073.55,7420.64");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2527.38,7271.28");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3568.98,7449.66");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4327.35,7468.37");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1484.34,7104.97");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3087.27,7165.64");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4048.31,7165.46");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4698.85,7205.53");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5130.56,7251.53");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5571.89,7107.60");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=639.80,6836.92");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1979.64,6856.71");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3733.36,6892.67");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4922.02,6917.82");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1064.30,6691.15");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1433.46,6685.67");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2629.36,6758.93");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2970.31,6602.28");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3508.46,6568.63");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4349.32,6566.53");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4855.18,6559.83");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5296.86,6672.26");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=154.11,6440.85");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1939.63,6381.42");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3922.95,6430.94");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4597.45,6305.64");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5686.69,6439.80");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=480.02,6196.22");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=861.39,6185.21");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1458.23,6171.73");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1835.04,6029.36");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2397.45,6266.17");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2865.63,6237.98");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3195.95,6056.69");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3614.43,6197.56");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4122.19,6033.39");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5336.51,6252.91");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5886.07,6083.10");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1139.20,5908.08");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2301.94,5850.99");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3594.05,5838.94");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4986.17,5898.95");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=283.38,5716.52");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=701.82,5657.54");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1532.39,5767.94");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4609.29,5544.45");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1235.29,5289.70");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1804.08,5488.35");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2243.65,5309.36");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2794.08,5471.20");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3171.62,5521.07");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3552.67,5416.79");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4152.23,5349.16");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5342.75,5467.46");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=683.34,5075.79");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2694.27,5130.26");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3125.35,5053.01");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3530.71,5050.27");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4527.63,5190.31");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4941.39,5263.81");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=398.46,4867.20");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1636.07,4896.16");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2180.29,4902.74");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3967.09,4963.31");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5494.51,4862.97");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=757.66,4723.34");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1327.88,4573.07");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2742.11,4641.26");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3360.95,4554.88");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4324.37,4552.26");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4953.14,4758.92");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5743.41,4593.94");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=7.21,4365.14");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=675.34,4329.76");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1885.29,4318.81");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2195.91,4496.74");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3847.67,4293.92");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=448.53,4047.67");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=983.24,4047.72");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2305.35,4098.95");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3027.54,4231.75");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3398.49,4100.19");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4254.78,4178.67");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5014.96,4283.63");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5599.62,4185.55");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1334.80,3890.61");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3925.70,3857.87");
    }

}

