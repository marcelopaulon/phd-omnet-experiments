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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=142.19,20.50");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=148.99,163.21");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=193.49,97.91");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=288.00,82.84");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=349.82,16.89");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=257.33,14.80");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=283.29,223.42");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=341.68,293.58");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=386.51,353.24");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=288.25,352.66");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=227.69,394.15");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=393.35,227.76");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=344.75,443.57");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=495.93,383.66");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=438.15,411.79");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=478.97,488.15");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=388.36,488.73");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=276.11,433.15");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=210.18,299.94");
    }

}

