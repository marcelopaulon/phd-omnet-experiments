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

network DADCA_ACK_n100_u8_a8000m2_5
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
        @display("i=misc/sensor;p=943.03,7817.32");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1690.31,7903.36");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2579.63,7843.53");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3133.50,7875.02");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3722.56,7823.63");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4852.96,7828.07");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5524.11,7825.60");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=274.27,7581.47");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1271.97,7553.41");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4237.19,7573.36");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=896.63,7266.73");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2993.25,7493.40");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3513.76,7427.81");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4652.80,7446.17");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5291.76,7350.01");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=61.08,7186.48");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=424.85,7170.97");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1606.30,7032.54");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2291.09,7050.32");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3808.85,7083.56");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1245.76,6886.95");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1934.52,6792.25");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3324.63,6962.96");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4351.82,6895.12");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4729.31,6975.52");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5222.76,6991.11");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=309.90,6683.33");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=737.80,6702.99");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2743.75,6631.34");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3689.59,6732.95");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5034.34,6666.33");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5638.94,6761.37");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=12.39,6482.41");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1261.68,6268.74");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1857.79,6414.46");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3209.33,6426.45");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4627.92,6284.62");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5911.75,6455.65");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=358.43,6172.30");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2359.75,6170.47");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2927.49,6159.04");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3511.54,6198.80");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4126.58,6150.23");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5233.65,6231.22");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5634.11,6113.92");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=557.11,5866.08");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=926.10,6017.95");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4964.43,5929.08");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=223.17,5635.83");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=845.62,5588.84");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1413.98,5771.93");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2059.01,5632.97");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2446.31,5736.77");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2872.17,5616.46");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3365.54,5605.95");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3744.53,5708.98");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4309.14,5576.47");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5582.76,5634.36");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=438.49,5278.65");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1258.66,5286.92");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3070.36,5289.11");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4547.56,5289.07");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4931.54,5496.86");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5835.56,5388.71");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=56.07,5242.31");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1596.02,5128.25");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2219.97,5097.14");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3381.91,5075.04");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5142.52,5173.05");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=788.63,4864.66");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2688.47,4838.98");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4361.17,4806.26");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5450.85,4917.21");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=74.53,4707.57");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=409.41,4561.32");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1435.13,4654.01");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3029.28,4740.67");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3835.84,4747.52");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4843.09,4772.53");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5898.85,4735.46");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1865.03,4354.81");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2310.33,4450.78");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2949.03,4293.58");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3449.92,4504.06");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4205.68,4351.68");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4645.48,4400.03");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5224.84,4345.99");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5713.60,4366.89");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=147.44,4106.70");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=898.73,4195.12");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1513.94,4236.48");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2149.80,4123.07");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=549.37,3992.80");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1905.77,3813.48");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2635.82,4012.04");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3387.63,3960.64");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3877.84,3981.74");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4285.91,3950.20");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4904.94,4000.47");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5712.50,3827.13");
    }

}

