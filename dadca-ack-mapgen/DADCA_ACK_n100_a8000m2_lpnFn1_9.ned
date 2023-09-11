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

network DADCA_ACK_n100_a8000m2_lpnFn1_9
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
        @display("i=misc/sensor,p=129.58,29.42");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=60.51,37.63");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=28.25,210.15");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=32.45,309.08");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=93.02,263.69");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=135.78,175.78");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=229.70,177.95");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=291.32,197.64");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=320.76,265.16");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=356.53,324.01");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=352.23,398.39");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=276.40,384.35");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=289.05,318.47");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=448.06,387.98");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=427.04,325.04");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=233.05,271.54");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=525.20,245.30");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=463.74,184.75");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=434.64,89.24");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=537.60,83.18");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=623.55,107.17");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=693.44,154.19");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=771.74,186.20");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=843.57,210.79");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=909.76,229.92");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=979.49,276.21");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=985.96,190.93");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=1064.51,200.38");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=1157.18,200.32");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=1234.49,232.76");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=1160.10,114.43");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=1210.19,163.69");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=1343.31,216.41");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=1318.58,277.78");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=1242.68,72.99");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=1279.15,177.38");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=1099.54,144.91");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=1038.39,103.10");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=1065.86,17.13");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=1272.71,9.44");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=1275.81,344.34");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=1364.53,366.73");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=1310.39,441.82");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=1376.83,495.83");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1435.71,459.33");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1500.99,384.15");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1461.32,545.92");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1448.30,623.95");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=1545.67,628.85");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=1564.66,699.76");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=1656.83,733.30");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=1636.28,800.54");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=1726.11,812.45");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=1798.23,846.22");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=1863.33,770.45");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=1946.37,783.48");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=2003.79,853.81");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=1914.98,868.16");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=1802.85,663.71");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=1613.53,866.90");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=1554.45,931.59");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=1531.47,873.56");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=1538.82,789.84");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=1498.08,710.71");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=1418.53,681.58");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=1335.10,715.31");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=1312.24,642.05");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=1366.89,585.17");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1226.94,524.66");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1153.32,513.11");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=1065.72,543.65");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=1026.24,476.80");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=1073.77,421.90");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=1098.57,349.47");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=998.91,387.22");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=1128.13,579.31");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=928.27,395.12");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=925.80,333.43");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=853.94,353.19");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=854.75,114.21");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=859.53,27.91");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=777.40,80.05");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=756.69,14.29");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=658.44,14.37");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=682.21,281.69");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=614.23,288.97");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=743.34,255.97");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=679.97,400.01");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=778.88,407.40");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=707.71,456.99");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=702.20,556.59");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=663.08,607.07");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=634.88,662.42");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=710.54,702.63");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=594.54,503.11");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=577.93,580.15");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=753.28,799.05");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=727.36,642.98");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=553.70,694.38");
    }

}

