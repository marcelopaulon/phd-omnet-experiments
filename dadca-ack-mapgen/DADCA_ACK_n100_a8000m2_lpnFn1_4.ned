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

network DADCA_ACK_n100_a8000m2_lpnFn1_4
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
        @display("i=misc/sensor,p=167.59,104.33");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=126.13,39.38");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=31.17,61.51");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=101.60,161.71");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=180.47,172.98");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=217.32,240.28");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=117.12,259.23");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=292.03,161.90");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=272.53,317.35");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=204.12,308.99");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=6.79,124.85");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=47.04,232.23");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=244.15,431.81");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=185.40,494.03");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=371.17,252.26");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=283.75,225.36");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=348.32,194.25");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=465.07,309.85");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=439.90,364.37");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=398.36,452.52");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=449.07,502.47");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=401.31,557.77");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=359.18,601.61");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=292.99,641.03");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=353.14,521.84");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=353.79,405.16");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=498.82,598.37");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=579.89,643.70");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=652.09,618.90");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=646.02,536.37");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=562.62,494.17");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=538.70,435.63");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=672.83,405.06");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=733.69,425.61");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=724.73,498.05");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=797.31,472.68");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=878.80,429.96");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=816.26,388.90");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=876.41,352.28");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=906.43,283.25");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=974.16,345.82");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=1060.43,374.42");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=1106.07,437.14");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=1161.55,482.26");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1213.15,553.44");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1269.03,518.05");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1269.10,591.66");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1325.64,673.64");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=1365.36,740.33");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=1434.87,739.78");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=1420.72,810.89");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=1361.37,857.48");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=1463.44,878.53");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=1331.97,802.31");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=1268.39,800.43");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=1225.39,720.90");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=1138.72,723.35");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=1144.10,651.78");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=1126.21,592.39");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=1170.69,385.22");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=1121.77,340.38");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=979.52,405.69");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=932.68,465.82");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=997.00,511.79");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=956.05,565.28");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=878.02,560.08");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=908.50,645.38");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=995.88,674.52");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1055.41,547.21");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1024.32,618.70");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=1068.12,703.67");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=1085.02,791.92");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=1150.25,828.38");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=1007.11,796.86");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=969.07,888.26");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=1071.69,851.52");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=838.27,718.47");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=847.18,806.39");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=786.86,835.44");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=847.01,901.34");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=780.41,928.72");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=933.12,943.60");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=1089.09,959.47");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=1027.41,956.42");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1140.02,915.85");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=1170.60,971.51");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=1085.30,1076.93");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=912.75,833.50");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=885.16,990.08");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=758.14,1020.77");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=688.05,1017.40");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=739.53,1112.71");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=825.05,1115.17");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=852.06,1200.19");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=891.24,1266.33");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=926.62,1112.53");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1018.41,1090.02");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=999.72,1192.03");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1027.66,442.96");
    }

}

