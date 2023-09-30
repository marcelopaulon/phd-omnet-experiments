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
        @display("i=misc/sensor;p=3526.56,7889.98");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=4427.18,7910.79");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4889.91,7972.70");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5720.11,7773.96");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1385.38,7516.59");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1763.84,7743.57");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2222.43,7719.36");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2715.11,7657.86");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3480.87,7521.28");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3977.97,7611.75");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4445.50,7538.38");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5300.04,7740.81");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=381.19,7402.28");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=954.56,7423.59");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1971.19,7373.62");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2399.23,7286.94");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=534.27,7015.74");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1466.46,7099.79");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3782.71,7164.45");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4354.88,7086.27");
    }

}

