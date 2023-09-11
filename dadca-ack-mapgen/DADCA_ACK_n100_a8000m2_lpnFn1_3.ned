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

network DADCA_ACK_n100_a8000m2_lpnFn1_3
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
        @display("i=misc/sensor,p=141.20,150.42");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=54.81,182.83");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=92.50,243.56");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=34.54,266.82");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=283.81,126.69");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=252.46,208.31");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=316.34,203.19");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=278.52,329.47");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=337.61,371.21");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=358.59,431.95");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=317.38,494.43");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=341.14,584.30");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=363.71,662.10");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=445.53,716.05");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=493.73,753.70");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=585.65,767.41");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=647.61,726.55");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=731.00,744.31");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=714.09,626.87");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=668.07,808.35");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=624.42,631.53");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=537.22,633.15");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=453.60,631.83");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=482.90,550.92");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=397.50,560.68");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=409.54,487.16");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=456.42,415.97");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=525.74,409.07");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=545.81,536.68");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=789.34,724.33");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=830.18,636.30");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=682.19,558.36");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=812.00,810.70");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=845.07,870.62");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=909.85,871.03");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=888.44,979.50");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=818.95,967.04");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=755.12,956.50");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=787.07,1063.89");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=843.16,1027.09");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=699.10,1072.69");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=614.77,1019.37");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=537.72,971.05");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=547.59,1057.52");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=529.71,1146.46");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=585.01,1105.34");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=660.46,1119.39");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=674.78,1203.02");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=608.80,1225.93");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=905.10,1068.73");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=971.75,1027.91");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=1049.30,1068.31");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=1087.81,1124.00");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=1115.43,1196.34");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=1088.63,1272.25");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=1059.23,1362.71");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=998.69,1401.72");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=956.67,1333.94");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=903.09,1418.71");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=827.70,1435.01");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=813.38,1371.28");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=815.48,1296.27");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=738.91,1254.72");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=983.76,1097.98");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=1006.70,1177.76");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=1118.15,939.14");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=1046.86,947.38");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=839.03,1121.73");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=876.11,1202.09");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=948.92,1218.78");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=789.18,1172.44");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=709.09,1001.42");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=634.22,935.99");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=582.23,887.85");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=511.20,818.52");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=455.30,935.77");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=451.78,869.59");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=375.16,969.03");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=313.93,1035.04");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=239.65,1060.09");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=306.73,1108.43");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=218.74,1126.46");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=244.58,1186.43");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=272.09,1274.25");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=336.76,1212.57");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=124.94,1255.72");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=79.43,1296.76");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=95.46,1375.52");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=35.29,1355.05");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=39.75,1428.01");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=135.49,1433.30");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=114.15,1496.40");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=133.94,1561.40");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=85.35,1607.45");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=106.22,1687.99");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=188.69,1684.81");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=150.63,1621.27");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=34.36,1643.59");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=53.46,1721.73");
    }

}

