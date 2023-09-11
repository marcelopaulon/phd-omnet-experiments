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
        @display("i=misc/sensor,p=106.40,25.62");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=178.48,50.04");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=12.26,15.56");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=7.93,83.00");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=45.01,136.81");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=38.46,228.70");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=24.08,306.70");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=27.43,397.55");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=162.61,272.53");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=230.16,301.61");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=295.45,300.13");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=356.05,290.76");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=433.79,334.54");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=444.20,415.22");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=374.54,379.09");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=431.82,514.87");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=365.99,550.88");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=304.66,519.95");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=382.43,475.63");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=539.21,479.48");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=489.50,532.79");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=551.40,569.54");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=538.31,402.86");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=314.37,426.99");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=410.96,653.54");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=483.95,634.73");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=595.00,351.01");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=602.98,424.46");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=625.82,518.00");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=657.77,593.04");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=663.38,367.72");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=735.12,327.63");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=713.31,267.78");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=636.59,291.66");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=539.37,318.79");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=525.75,226.87");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=446.73,253.11");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=581.28,164.44");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=640.95,204.91");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=781.61,193.23");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=724.43,165.68");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=802.99,379.98");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=882.81,233.24");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=907.79,164.20");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=905.50,305.91");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=990.97,283.81");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1022.37,352.63");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1065.63,282.50");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=1038.70,211.18");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=1088.86,342.38");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=1186.06,329.28");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=1244.53,276.40");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=1269.75,191.89");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=1334.51,157.70");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=1405.13,173.72");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=1470.50,198.88");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=1518.77,127.21");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=1579.95,143.13");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=1538.87,203.84");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=1636.33,232.66");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=1611.39,303.98");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=1420.79,315.37");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=1436.31,397.60");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=1487.09,433.22");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=1484.65,499.66");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=1577.28,464.56");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=1479.14,570.83");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=1427.58,632.93");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1577.01,547.04");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1594.99,608.67");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=1680.36,600.01");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=1724.12,556.78");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=1646.82,503.62");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=1628.28,671.32");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=1544.87,667.66");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=1301.11,585.85");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=1375.49,547.51");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=1810.46,599.33");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=1849.06,538.02");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=1915.62,524.55");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=1954.30,467.35");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=2010.64,498.99");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=2003.52,351.34");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=2001.09,258.82");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1937.78,277.33");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=1970.38,200.84");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=1989.21,140.93");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=2071.91,136.31");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=2142.96,72.40");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=2180.79,141.98");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=2190.97,232.58");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=2118.85,237.91");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=2044.81,82.49");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=2104.25,3.87");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=2224.75,97.86");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=2062.89,199.30");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=2282.55,173.72");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=2325.33,131.13");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=2324.83,44.11");
    }

}

