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

network DADCA_ACK_n20_u2_a8000m2_0
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
        @display("i=misc/sensor;p=229.30,7753.62");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=779.24,7935.49");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2726.62,7920.14");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3203.68,7821.89");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3872.24,7827.50");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4672.40,7983.44");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5191.58,7787.95");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1509.27,7692.99");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1872.95,7647.70");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2354.55,7612.85");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3591.11,7514.09");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4479.66,7640.05");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5512.52,7598.54");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=624.61,7368.14");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1160.52,7489.04");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2272.82,7267.04");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2729.81,7381.61");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3971.46,7423.52");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4536.95,7269.44");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4953.13,7400.24");
    }

}

