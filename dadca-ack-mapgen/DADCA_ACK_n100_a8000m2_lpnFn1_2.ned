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

network DADCA_ACK_n100_a8000m2_lpnFn1_2
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
        @display("i=misc/sensor,p=15.32,101.56");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=59.57,27.78");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=196.51,73.01");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=258.15,106.05");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=278.98,201.88");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=351.07,248.08");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=369.22,177.47");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=391.49,86.40");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=462.64,135.38");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=519.45,94.20");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=518.13,7.15");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=426.95,37.84");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=577.12,173.92");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=607.69,92.98");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=593.26,25.93");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=640.52,172.77");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=549.61,261.55");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=648.84,273.43");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=279.22,262.16");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=222.44,17.01");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=294.93,2.10");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=132.41,1.43");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=86.28,198.59");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=17.44,201.58");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=190.44,198.19");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=519.98,198.20");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=428.59,206.43");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=362.49,348.21");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=448.47,395.68");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=394.91,433.61");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=285.15,323.75");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=218.41,389.59");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=280.10,411.09");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=218.79,481.12");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=25.97,314.05");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=91.99,376.58");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=168.60,423.86");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=403.10,281.75");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=684.43,121.10");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=743.49,167.78");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=665.97,44.93");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=580.35,371.28");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=502.26,360.70");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=671.28,376.68");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=444.86,329.81");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=175.70,278.41");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=321.19,125.45");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=332.27,59.52");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=309.80,503.74");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=381.68,533.95");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=263.14,555.96");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=96.77,308.89");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=61.53,259.20");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=31.97,449.97");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=13.65,382.74");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=138.42,484.92");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=77.99,549.89");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=127.15,629.37");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=189.90,635.59");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=203.31,699.04");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=14.14,691.60");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=40.81,780.44");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=16.77,836.36");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=330.66,443.74");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=510.64,485.38");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=451.40,507.52");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=470.89,573.28");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=469.02,641.61");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=527.47,700.51");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=584.34,725.35");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=609.67,668.16");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=677.93,816.26");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=740.50,835.54");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=814.71,831.53");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=857.94,783.11");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=949.65,793.44");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=942.80,727.61");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=982.28,665.31");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=1037.66,695.63");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=1120.47,701.03");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=1072.13,626.76");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=1050.45,557.07");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=1160.60,582.82");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=1175.51,743.55");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1231.14,697.33");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=1204.07,631.43");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=1272.20,571.23");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=1346.43,532.79");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=1427.97,503.90");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=1209.19,490.77");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1120.15,500.09");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1332.65,473.84");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1360.19,609.27");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1432.05,578.17");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1309.48,699.02");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1283.24,782.36");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1227.50,837.95");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1354.26,822.67");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1413.89,888.84");
    }

}

