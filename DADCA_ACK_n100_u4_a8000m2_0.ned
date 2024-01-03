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

network DADCA_ACK_n100_u4_a8000m2_0
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
        @display("i=misc/sensor;p=778.03,7926.07");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1467.75,7897.09");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2129.61,7852.80");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2691.49,7853.87");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4493.19,7994.54");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5008.58,7825.99");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5451.99,7930.34");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3089.16,7659.83");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3458.19,7607.29");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3922.42,7594.15");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4596.23,7547.66");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5849.81,7736.28");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=248.70,7498.19");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=922.32,7385.12");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1504.20,7383.62");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2719.10,7396.00");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4176.04,7259.81");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5154.02,7437.27");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5731.88,7382.27");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2001.45,7031.46");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2305.73,7241.77");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3268.48,7118.87");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5009.33,7040.47");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=606.03,6937.27");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=940.17,6791.43");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1341.57,6842.34");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2598.74,6985.73");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3120.07,6788.68");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3615.38,6977.43");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5521.32,6905.37");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=248.49,6645.01");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1971.62,6562.41");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2567.22,6635.93");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3561.66,6515.37");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4035.19,6554.47");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4489.12,6585.14");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4852.82,6643.78");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5289.03,6535.29");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5893.92,6734.74");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1689.30,6353.54");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2964.50,6432.88");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5891.46,6337.14");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=335.22,6024.57");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=608.78,6250.90");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1095.68,6241.46");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2109.89,6109.05");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3523.19,6070.83");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3918.57,6188.37");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4727.95,6264.33");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5474.14,6219.73");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1329.32,5948.86");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1800.55,5941.88");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2858.72,5912.37");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4184.59,5936.27");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4893.86,5798.72");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=704.98,5724.17");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1345.08,5556.50");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2442.27,5705.42");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2937.12,5548.25");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3270.65,5740.56");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4342.05,5622.64");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5342.43,5740.09");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5692.88,5681.68");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=980.26,5490.32");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1791.49,5477.31");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3831.57,5438.71");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4889.60,5361.51");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5581.02,5297.88");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=650.78,5047.18");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1098.95,5108.67");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1750.16,5033.69");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2599.83,5116.38");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3108.41,5110.22");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3440.20,5226.41");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3861.80,5067.92");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4522.74,5130.77");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5932.57,5207.32");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=102.87,4875.03");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1469.08,4818.16");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2208.41,4917.25");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5229.54,4794.54");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5621.44,4891.43");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=424.99,4669.58");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1906.46,4628.99");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3562.10,4576.05");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4107.41,4600.34");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4884.60,4722.09");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5888.12,4647.77");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=82.16,4337.14");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=872.49,4500.28");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1299.03,4491.45");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2236.98,4323.33");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2827.98,4490.09");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4599.62,4504.44");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5634.52,4327.90");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=427.90,4243.22");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1076.38,4144.19");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1685.81,4121.96");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3331.96,4149.19");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3841.68,4178.14");
    }

}

