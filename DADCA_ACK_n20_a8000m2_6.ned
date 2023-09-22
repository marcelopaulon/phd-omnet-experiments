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

network DADCA_ACK_n20_a8000m2_6
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
        @display("i=misc/sensor;p=973.45,7873.78");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2122.12,7887.67");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2595.70,7940.19");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2972.03,7838.99");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5644.11,7777.31");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=286.22,7565.13");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1909.81,7554.49");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2677.55,7490.54");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4070.95,7581.19");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4770.23,7693.08");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=913.18,7460.33");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1427.86,7467.37");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2334.55,7282.70");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4213.47,7235.33");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4603.13,7279.17");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5319.07,7227.01");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5835.22,7257.23");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=17.03,7111.34");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=381.50,6971.13");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=837.74,6993.14");
    }

}

