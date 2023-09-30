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

network DADCA_ACK_n20_u3_a8000m2_9
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
        @display("i=misc/sensor;p=636.01,7822.70");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1622.65,7940.71");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2296.14,7959.24");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3281.20,7913.89");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4137.02,7785.13");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5071.10,7857.15");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5752.05,7794.72");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=168.49,7700.37");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=951.77,7669.36");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1295.24,7546.04");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1865.27,7614.70");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2628.01,7709.96");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=636.99,7433.64");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3227.36,7489.44");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3763.57,7502.00");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4566.52,7388.02");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4916.70,7291.76");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5377.29,7480.56");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5985.80,7333.38");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=387.20,7133.34");
    }

}

