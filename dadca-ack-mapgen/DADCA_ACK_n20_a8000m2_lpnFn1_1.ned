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

network DADCA_ACK_n20_a8000m2_lpnFn1_1
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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=101.13,164.71");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=44.26,188.83");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=104.78,232.75");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=179.55,173.70");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=170.41,113.82");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=264.39,146.61");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=265.89,242.56");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=356.07,240.18");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=367.06,304.06");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=282.57,309.14");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=243.87,383.56");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=239.99,463.13");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=319.35,522.59");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=331.52,602.84");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=398.32,452.25");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=386.70,654.40");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=342.30,723.68");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=434.12,717.30");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=516.94,671.05");
    }

}

