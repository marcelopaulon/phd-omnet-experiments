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

network DADCA_ACK_n20_u4_a8000m2_7
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
        @display("i=misc/sensor;p=1216.14,7870.94");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1861.13,7989.95");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2530.91,7801.68");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3150.10,7821.73");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4098.22,7785.36");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5000.48,7968.70");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5609.17,7926.51");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=168.36,7738.34");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=667.42,7686.05");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1731.60,7607.33");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2216.61,7564.45");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2832.99,7549.11");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3512.50,7602.91");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=76.43,7395.58");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1268.67,7480.06");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4846.15,7479.58");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5265.30,7448.62");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5809.86,7429.50");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=447.97,7164.39");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1780.83,7230.90");
    }

}

