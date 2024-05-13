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

network DADCA_ACK_n20_u2_a8000m2_9
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
        @display("i=misc/sensor;p=3108.85,7914.01");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=4847.98,7952.46");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=5255.39,7847.58");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5671.57,7865.69");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=785.51,7725.58");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1381.55,7666.56");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1977.81,7668.06");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2568.14,7662.09");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3757.95,7655.67");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4709.15,7621.65");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=8.28,7392.33");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=460.51,7478.70");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=996.69,7323.26");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2167.12,7340.71");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2934.58,7476.68");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3310.85,7448.00");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4323.16,7403.59");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5084.25,7485.90");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5501.51,7417.81");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=693.76,7053.70");
    }

}

