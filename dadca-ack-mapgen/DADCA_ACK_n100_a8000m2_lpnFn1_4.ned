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
        @display("i=misc/sensor,p=152.69,180.64");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=209.67,224.96");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=215.59,151.75");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=185.49,319.29");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=256.15,331.39");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=320.19,276.65");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=393.77,319.65");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=432.70,369.28");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=487.91,433.52");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=463.62,502.44");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=412.67,430.81");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=443.21,274.51");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=417.12,202.37");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=546.51,294.92");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=497.54,216.43");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=522.32,350.53");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=335.58,200.40");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=381.60,140.53");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=296.60,87.15");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=600.08,405.32");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=681.13,354.90");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=740.98,403.81");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=727.84,487.63");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=846.88,444.68");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=833.22,530.24");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=826.76,363.90");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=900.28,337.42");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=983.30,322.76");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=937.76,235.88");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=872.62,183.11");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=836.37,109.60");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=856.23,38.54");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=767.67,34.90");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=703.13,31.42");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=619.39,83.22");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=542.61,38.99");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=479.37,23.29");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=466.07,86.92");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=419.07,40.08");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=527.31,98.25");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=579.83,242.98");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=658.19,199.34");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=724.62,127.25");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=769.89,251.53");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=971.58,184.53");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=983.47,93.99");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1018.65,27.00");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1055.24,106.60");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=1136.33,162.98");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=1201.80,184.07");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=1252.00,96.19");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=1238.73,4.95");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=1333.66,28.09");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=1370.53,117.77");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=1260.00,160.99");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=1214.78,281.02");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=1147.05,339.25");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=1073.55,331.08");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=1095.88,407.92");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=1031.70,236.42");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=1135.07,102.05");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=1092.19,230.16");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=1086.90,47.07");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=1166.91,443.22");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=1227.86,340.23");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=1259.14,407.79");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=1235.75,480.09");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=1146.61,510.22");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1258.65,234.24");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1290.78,321.72");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=907.41,80.31");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=796.16,310.91");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=1257.08,615.69");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=1293.55,567.25");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=1189.00,699.67");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=1275.85,746.96");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=1240.73,829.81");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=1328.75,866.62");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=1401.76,898.56");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=1388.75,974.76");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=1458.06,958.62");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=1509.81,902.71");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=1569.30,918.99");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=1566.85,1009.04");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1656.32,881.28");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=1628.31,796.08");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=1574.29,746.62");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=1543.84,650.23");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=1502.55,704.48");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=1520.88,524.59");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1603.48,529.83");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1606.03,450.61");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1699.17,422.48");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1763.41,459.33");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1777.66,522.38");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1726.84,559.23");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1793.64,582.67");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1834.42,633.56");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1707.38,627.93");
    }

}

