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

network DADCA_ACK_n100_u16_a8000m2_8
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
        @display("i=misc/sensor;p=194.07,7858.84");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1191.52,7961.59");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1806.25,7873.76");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3120.55,7756.18");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3413.28,7962.77");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3788.77,7934.23");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4215.22,7991.23");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5117.64,7808.89");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=846.66,7642.65");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1485.92,7636.56");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2288.85,7564.25");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2780.09,7655.74");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4305.70,7601.59");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4770.86,7518.28");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5416.67,7557.94");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=229.58,7310.87");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3266.15,7330.67");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3850.37,7427.95");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=551.10,7050.19");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1029.26,7256.06");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1969.86,7213.61");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2505.87,7237.98");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2919.43,7174.90");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3842.29,7029.15");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4273.30,7106.25");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5038.27,7253.84");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5365.66,7030.79");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5707.44,7183.89");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1393.42,6855.99");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3500.30,6937.52");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4721.40,6822.67");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=299.92,6705.16");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=750.44,6679.25");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2013.79,6698.87");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2497.42,6696.00");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3998.14,6574.41");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5311.94,6608.58");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5718.14,6605.77");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1109.33,6311.89");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1517.87,6385.73");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2211.06,6350.17");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3012.69,6399.31");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3503.89,6474.71");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=17.23,6218.03");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=543.28,6119.17");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2677.01,6129.57");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3368.71,6070.37");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4028.99,6125.87");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5177.85,6255.53");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=336.12,5826.42");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1435.07,5859.43");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1992.33,5810.48");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4664.58,6003.48");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5282.73,5834.90");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5668.22,5980.71");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5961.36,5787.25");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=873.49,5732.63");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3155.46,5559.45");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4808.59,5648.97");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=498.16,5346.04");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1137.76,5391.15");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1510.86,5387.92");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1938.07,5291.02");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2385.52,5496.45");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2779.63,5471.28");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3440.42,5285.12");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4474.62,5351.04");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5437.49,5365.19");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1613.34,5050.84");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2228.47,5069.74");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3067.81,5173.13");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5052.38,5262.95");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5743.17,5063.37");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=46.11,4947.16");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1031.03,4890.89");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2700.82,4861.82");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3329.58,4887.45");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4122.20,4848.02");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4751.73,4879.63");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5303.50,4933.48");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=508.86,4696.81");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1340.99,4573.80");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1770.82,4627.87");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3782.46,4693.83");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5946.63,4563.68");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=209.19,4387.20");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=993.91,4431.96");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2746.08,4449.40");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3353.08,4302.54");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4654.40,4525.04");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5016.45,4469.90");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5599.96,4397.97");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1607.15,4108.24");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2278.18,4188.44");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2859.13,4088.37");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3785.49,4218.04");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4204.28,4165.26");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4678.51,4107.11");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=112.02,4014.49");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=548.05,4009.26");
    }

}

