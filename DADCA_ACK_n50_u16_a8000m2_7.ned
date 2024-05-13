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

network DADCA_ACK_n50_u16_a8000m2_7
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
        @display("i=misc/sensor;p=1147.04,7994.74");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1526.42,7966.08");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2177.83,7753.49");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2846.98,7947.28");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3208.71,7850.91");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4579.83,7883.90");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=780.79,7695.33");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1731.52,7639.91");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3539.21,7669.35");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5254.87,7739.49");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=384.71,7331.59");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1166.13,7453.66");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2798.84,7333.44");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3154.42,7474.25");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3956.02,7319.71");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4853.14,7487.57");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5772.30,7318.29");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1411.94,7089.34");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1897.12,7036.80");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2383.00,7210.97");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4448.75,7141.79");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5248.44,7160.87");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=274.84,6944.28");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=831.81,6893.37");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2833.42,6981.05");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3295.17,6916.63");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3918.32,6929.84");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4777.97,6794.52");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5793.37,6942.82");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=956.30,6531.33");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1638.78,6734.03");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2309.96,6712.92");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3280.07,6565.24");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4214.08,6640.37");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5381.68,6757.67");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=295.54,6494.89");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1414.78,6417.63");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2084.90,6353.08");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2806.12,6353.20");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3675.67,6409.26");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4337.73,6304.48");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4765.69,6405.24");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5701.13,6466.73");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1124.76,6038.80");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1701.69,6164.65");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4599.78,6031.72");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5090.07,6213.35");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5897.37,6084.29");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=329.70,5981.63");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=785.03,5926.35");
    }

}

