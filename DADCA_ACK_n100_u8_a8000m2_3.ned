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

network DADCA_ACK_n100_u8_a8000m2_3
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
        @display("i=misc/sensor;p=908.38,7897.58");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1958.95,7970.90");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2449.63,7914.74");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3589.58,7824.92");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5103.34,7812.55");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5580.33,7983.35");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5966.99,7961.12");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=575.69,7619.74");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1822.84,7547.28");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2165.05,7632.93");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2935.57,7692.55");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4158.96,7695.90");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5602.52,7624.28");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=352.55,7324.72");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1192.90,7465.47");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2440.83,7347.71");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3395.40,7416.72");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5276.53,7402.48");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2941.24,7050.77");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3837.27,7199.94");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4480.08,7156.15");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4941.05,7217.65");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5845.71,7132.46");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=37.32,6764.07");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=487.88,6839.08");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=805.40,7009.33");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1277.32,6808.65");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1760.35,6962.87");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2241.44,7004.75");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4163.82,6914.29");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5212.14,6764.11");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5506.46,6978.85");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2368.77,6608.91");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3367.23,6691.82");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4219.47,6560.89");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4680.40,6587.37");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=137.01,6310.10");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=580.93,6436.00");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1328.34,6269.99");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1711.50,6399.64");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2085.20,6319.42");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3066.67,6436.51");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3739.37,6329.92");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5621.35,6472.58");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=933.77,6114.21");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1723.05,6038.60");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2651.88,6161.20");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4141.48,6129.96");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4753.89,6151.69");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5235.56,6176.80");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=331.49,5830.15");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2262.41,5980.82");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3251.67,5990.83");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4489.68,5865.48");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5557.77,6004.18");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1005.43,5740.19");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1591.47,5540.95");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2579.58,5575.12");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3483.13,5555.65");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3996.37,5689.88");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5061.09,5620.49");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=49.66,5383.43");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=998.35,5334.03");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2019.90,5489.96");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3014.52,5405.73");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4550.76,5295.39");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5669.69,5463.96");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=658.35,5225.56");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1685.46,5035.76");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2395.69,5129.48");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3955.81,5148.11");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4921.15,5092.61");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5531.60,5084.77");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=799.03,4834.28");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1172.14,4891.34");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3036.88,4877.94");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3622.99,4880.56");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4297.59,4795.30");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5968.94,5029.30");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1378.36,4545.84");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1752.93,4643.83");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2282.18,4560.29");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5183.45,4726.25");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=981.54,4362.78");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3146.22,4473.23");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3673.87,4448.06");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4038.51,4416.28");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4738.59,4459.10");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5866.74,4492.38");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=442.12,4199.06");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1274.28,4136.06");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1823.20,4060.66");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3100.28,4072.10");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3707.73,4069.78");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5130.36,4260.66");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5593.27,4234.28");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=723.80,3828.61");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1120.01,3815.71");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2240.84,3857.14");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4367.27,4037.20");
    }

}

