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

network DADCA_ACK_n20_a8000m2_5
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
        @display("i=misc/sensor;p=105.00,7966.39");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=164.98,7998.95");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=367.91,7977.51");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=439.54,7992.85");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=523.18,7985.53");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=626.85,7996.55");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=732.68,7983.27");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=858.78,7997.03");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=980.66,7969.80");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=22.91,7952.88");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=245.69,7956.49");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=299.99,7914.66");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=447.40,7923.45");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=558.90,7931.46");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=679.77,7952.59");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=920.19,7927.85");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=92.09,7906.46");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=154.69,7911.44");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=619.32,7911.50");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=718.12,7888.81");
    }

}

