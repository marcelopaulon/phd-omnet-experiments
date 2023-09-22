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

network DADCA_ACK_n20_a8000m2_8
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
        @display("i=misc/sensor;p=150.59,7947.23");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1170.17,7924.97");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1649.45,7990.57");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2176.47,7978.28");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3110.42,7959.43");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4065.72,7842.56");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4456.23,7775.13");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5119.23,7824.14");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5580.94,7926.22");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=480.49,7505.91");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=839.69,7703.96");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1468.29,7661.98");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1865.10,7583.76");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2512.52,7685.12");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2874.20,7488.52");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3721.09,7618.92");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5658.42,7503.01");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=29.89,7419.82");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3349.05,7238.76");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4298.45,7381.97");
    }

}

