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

network DADCA_ACK_n100_a8000m2_lpnFn1_6
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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=174.17,43.13");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=231.47,113.50");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=160.46,123.49");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=216.57,176.84");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=304.86,43.14");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=297.45,148.60");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=231.33,9.59");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=281.68,221.60");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=387.45,32.65");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=410.79,128.49");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=476.01,178.62");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=502.22,124.46");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=547.75,192.68");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=451.76,77.62");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=379.81,217.18");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=228.87,262.54");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=141.64,279.95");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=139.02,347.07");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=60.67,328.86");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=12.70,272.08");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=9.81,174.69");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=5.45,86.70");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=86.96,177.87");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=306.86,298.51");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=230.59,349.32");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=409.28,374.45");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=474.70,426.01");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=503.37,497.32");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=554.89,459.98");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=615.59,525.72");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=621.18,438.86");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=645.21,367.15");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=553.21,376.93");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=745.93,361.51");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=789.97,318.76");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=860.18,316.56");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=848.06,225.38");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=901.94,253.20");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=717.50,424.05");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=645.78,281.53");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=712.69,306.00");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=557.98,267.51");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=519.37,326.78");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=439.22,275.40");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=355.84,407.85");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=114.28,405.65");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=43.07,451.00");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=80.73,240.92");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=183.56,402.78");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=149.29,193.63");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=301.82,436.30");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=139.37,472.44");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=569.18,105.44");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=546.89,39.31");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=736.92,247.99");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=761.95,173.88");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=707.88,127.58");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=647.70,180.70");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=696.57,39.68");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=827.34,67.23");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=764.45,24.06");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=636.32,41.33");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=854.66,138.87");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=940.31,83.18");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=819.60,381.16");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=930.09,191.85");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=959.14,281.34");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=303.97,366.71");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=191.49,593.21");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=260.25,624.94");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=121.47,619.83");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=176.41,656.44");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=142.61,736.80");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=108.69,685.28");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=320.43,654.88");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=407.88,648.79");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=422.36,572.45");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=240.65,725.26");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=356.34,552.78");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=353.03,480.28");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=28.52,4.04");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=7.82,374.32");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=294.65,506.85");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=802.36,486.25");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=874.26,551.11");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=806.73,596.29");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=889.91,642.85");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=932.82,586.17");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=962.93,658.19");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=962.20,725.54");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1053.71,690.19");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1079.41,770.32");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=983.75,809.61");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=916.86,773.56");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=935.96,859.67");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1042.11,829.95");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=874.78,938.88");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=992.83,894.66");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1112.97,712.30");
    }

}

