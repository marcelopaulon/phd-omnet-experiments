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

network DADCA_ACK_n100_a8000m2_lpnFn1_8
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
        @display("i=misc/sensor,p=130.54,153.01");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=40.66,170.21");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=228.28,169.19");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=270.88,100.70");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=196.37,43.90");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=111.92,38.88");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=169.67,99.83");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=178.19,286.78");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=221.82,354.84");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=307.89,340.58");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=359.30,267.10");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=400.20,339.02");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=394.88,457.96");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=411.23,211.01");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=474.72,165.21");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=447.01,102.35");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=383.26,126.74");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=392.12,59.00");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=501.04,237.18");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=480.42,319.72");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=467.10,446.44");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=529.09,442.03");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=556.02,504.92");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=460.39,527.05");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=519.86,582.22");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=556.54,674.50");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=474.06,715.51");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=563.42,735.32");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=523.13,783.90");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=605.23,826.04");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=544.40,870.62");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=476.10,937.31");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=544.23,974.93");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=483.92,997.15");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=432.54,1110.93");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=539.51,1112.60");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=480.05,1202.22");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=546.00,1213.65");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=533.18,1307.72");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=608.46,1330.65");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=659.80,1279.54");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=716.72,1408.31");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=734.07,1332.01");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=768.95,1255.61");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=694.41,1213.70");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=643.29,1165.12");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=763.23,1189.49");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=720.98,1095.64");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=624.36,1105.50");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=765.04,1043.67");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=751.23,953.89");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=675.45,1016.49");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=645.67,936.52");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=684.62,888.65");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=686.48,805.10");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=744.62,835.25");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=836.13,855.85");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=821.08,950.35");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=426.54,793.95");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=343.73,762.71");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=307.64,849.29");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=260.60,929.48");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=241.64,823.84");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=221.29,763.18");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=275.08,687.02");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=205.76,700.15");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=137.19,756.34");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=74.16,806.30");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=228.60,598.40");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=130.89,621.50");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=165.65,538.08");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=289.52,592.27");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=79.12,705.03");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=7.89,766.91");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=169.76,910.25");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=104.24,933.79");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=214.82,1020.61");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=287.04,1003.92");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=324.35,956.78");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=358.40,886.73");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=393.08,953.12");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=362.90,1023.08");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=419.50,870.97");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=532.81,1035.93");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=623.47,766.52");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=630.37,703.35");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=483.93,847.01");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=401.90,706.11");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=619.19,613.21");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=702.09,624.48");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=711.50,551.58");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=638.35,553.51");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=643.66,493.63");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=712.43,471.11");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=795.79,471.82");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=802.57,397.99");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=806.13,613.61");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=812.64,553.91");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=859.84,514.15");
    }

}

