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

network DADCA_ACK_n100_a8000m2_lpnFn1_0
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
        @display("i=misc/sensor,p=115.75,29.06");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=225.89,116.37");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=172.52,73.88");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=30.04,81.20");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=39.26,19.45");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=137.52,160.81");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=260.27,216.99");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=208.79,288.35");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=116.50,262.59");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=329.86,113.40");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=412.85,120.46");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=430.85,56.02");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=429.05,212.36");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=502.75,241.32");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=455.54,304.07");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=476.69,125.19");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=360.40,308.31");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=400.97,370.42");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=348.73,450.89");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=423.97,442.03");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=509.92,418.11");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=501.31,353.25");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=575.11,382.11");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=634.70,431.80");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=415.35,518.85");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=354.83,593.86");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=331.57,669.03");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=303.48,733.65");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=255.83,687.34");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=226.95,622.19");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=161.90,572.90");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=70.60,601.53");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=92.48,509.87");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=93.16,424.63");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=1.46,433.15");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=40.04,341.66");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=65.42,221.42");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=0.58,216.49");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=52.04,137.15");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=111.69,353.60");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=158.91,434.57");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=222.53,389.83");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=248.89,448.04");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=224.41,532.82");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=303.71,409.68");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=199.58,174.43");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=124.88,629.61");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=40.11,748.03");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=73.76,804.16");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=136.88,813.99");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=169.23,679.66");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=59.66,673.89");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=218.44,0.64");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=286.33,3.16");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=375.91,21.55");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=342.15,212.53");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=285.37,71.51");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=287.57,555.05");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=297.83,282.26");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=219.93,752.39");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=598.19,220.21");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=678.81,210.56");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=714.87,142.14");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=643.53,154.84");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=808.72,155.63");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=742.05,217.93");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=750.61,306.99");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=772.78,366.12");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=719.02,412.40");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=789.95,427.65");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=634.22,322.02");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=531.64,474.88");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=473.33,500.76");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=497.02,562.02");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=577.98,595.26");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=699.07,680.50");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=753.61,635.94");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=825.33,635.71");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=819.99,572.38");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=838.35,504.34");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=652.91,515.11");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=583.70,508.72");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=566.11,668.78");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=597.97,766.46");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=529.78,755.31");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=471.66,699.96");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=423.75,783.08");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=387.67,715.98");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=317.39,847.62");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=295.46,934.79");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=221.95,849.56");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=392.01,834.11");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=173.79,924.28");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=380.61,951.13");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=461.87,966.38");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=555.51,964.42");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=505.34,892.36");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=561.78,865.12");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=483.84,795.48");
    }

}

