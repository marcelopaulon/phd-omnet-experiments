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

network DADCA_ACK_n50_u8_a8000m2_2
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
        @display("i=misc/sensor;p=1237.57,7953.43");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3037.77,7938.58");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3885.74,7782.29");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4440.21,7770.16");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5113.16,7820.86");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5691.27,7787.28");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=594.21,7627.87");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1589.06,7728.68");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2383.50,7588.37");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3412.45,7547.75");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1190.18,7463.65");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2038.04,7478.67");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2819.36,7473.13");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4077.45,7464.55");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4743.37,7415.79");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5097.41,7454.61");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=746.48,7087.88");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1476.03,7214.31");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1883.87,7163.95");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2408.97,7084.85");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2772.06,7119.50");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3888.91,7099.08");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4804.21,7062.70");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5674.24,7110.77");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=379.17,6822.35");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1052.46,6851.35");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1467.12,6860.93");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3387.73,6916.20");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4281.33,6851.79");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5233.69,6919.07");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=792.98,6610.01");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1948.85,6746.39");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3074.20,6643.13");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3794.03,6746.50");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4905.30,6580.62");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5907.40,6739.29");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=150.61,6480.47");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1056.77,6377.17");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1617.62,6492.47");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2344.91,6384.31");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3464.70,6433.55");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4057.01,6400.43");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4436.61,6360.25");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5601.53,6455.19");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=673.24,6168.96");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1870.42,6118.72");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3998.84,6044.59");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=232.92,6007.95");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1199.84,6014.12");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2387.88,5987.01");
    }

}

