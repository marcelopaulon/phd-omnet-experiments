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

network DADCA_ACK_n100_a8000m2_lpnFn1_5
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
        @display("i=misc/sensor,p=51.82,53.64");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=79.94,169.55");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=37.47,213.35");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=193.27,145.10");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=230.90,236.99");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=240.83,107.57");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=135.90,210.87");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=249.60,33.34");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=303.94,190.55");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=336.25,251.30");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=403.53,318.25");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=468.39,373.01");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=530.73,380.69");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=520.65,298.00");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=524.03,219.90");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=448.94,265.32");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=320.57,349.37");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=363.06,409.10");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=188.01,318.17");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=148.35,367.34");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=89.70,425.03");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=158.65,433.78");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=122.26,526.55");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=69.59,489.53");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=53.94,556.68");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=149.49,602.42");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=93.53,291.88");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=4.96,309.12");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=212.19,400.99");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=246.83,450.57");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=299.79,422.03");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=365.17,470.06");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=437.48,456.67");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=554.52,466.20");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=401.49,564.88");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=330.43,562.21");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=501.78,618.07");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=561.94,561.98");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=551.73,718.94");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=614.15,704.15");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=550.60,828.88");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=552.87,655.97");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=606.22,774.80");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=478.80,774.69");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=460.27,691.64");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=672.76,622.81");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=606.31,626.07");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=747.95,718.69");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=795.93,774.50");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=704.24,790.09");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=707.23,873.46");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=766.06,833.63");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=771.60,931.35");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=681.03,968.00");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=608.48,931.74");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=687.84,701.03");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=745.13,631.64");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=814.35,657.56");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=821.02,896.69");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=625.23,562.77");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=891.07,724.74");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=667.91,1073.31");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=728.30,1012.41");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=828.77,959.59");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=626.59,1010.09");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=617.76,1114.36");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=536.67,1056.96");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=516.09,1135.81");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=526.84,966.44");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=467.23,1015.83");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=392.56,1070.78");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=383.63,1005.68");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=303.52,991.08");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=340.31,1110.52");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=422.93,949.02");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=613.00,863.47");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=466.29,844.72");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=556.50,1188.63");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=645.83,1221.62");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=457.94,1076.87");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=406.46,891.01");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=415.35,624.54");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=458.82,513.80");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=356.87,715.18");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=279.19,724.30");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=283.86,801.99");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=186.15,664.09");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=267.86,635.19");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=249.13,545.56");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=736.06,539.10");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=697.04,484.11");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=642.27,413.69");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=592.26,327.22");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=664.57,299.20");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=465.62,124.18");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=559.01,158.69");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=602.20,229.91");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=599.59,23.43");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=530.57,70.79");
    }

}

