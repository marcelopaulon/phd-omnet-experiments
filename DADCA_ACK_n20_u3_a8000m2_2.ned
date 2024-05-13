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

network DADCA_ACK_n20_u3_a8000m2_2
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
        @display("i=misc/sensor;p=1209.11,7935.13");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2504.76,7884.22");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3496.23,7911.81");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4003.30,7957.25");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=243.67,7612.40");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=649.02,7744.32");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1849.71,7677.05");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2565.89,7512.42");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2987.63,7715.91");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3329.49,7547.96");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3889.24,7563.03");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4343.22,7553.69");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=457.11,7334.79");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1006.67,7444.99");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1605.82,7425.36");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2958.41,7271.40");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=645.36,7030.61");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1932.34,7025.76");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2320.72,7146.70");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3487.72,7052.75");
    }

}

