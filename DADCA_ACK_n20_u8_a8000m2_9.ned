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

network DADCA_ACK_n20_u8_a8000m2_9
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
        @display("i=misc/sensor;p=1124.18,7993.41");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1606.93,7895.37");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2758.92,7771.84");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4055.15,7947.75");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4510.68,7928.78");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4902.81,7868.74");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5426.00,7860.18");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=369.73,7708.76");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1146.89,7591.01");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2189.33,7751.53");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5046.66,7517.73");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5756.68,7599.02");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=274.41,7319.72");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=837.05,7425.69");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2109.47,7344.46");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2672.46,7432.05");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3350.39,7490.17");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4013.73,7471.01");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4556.43,7351.15");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1322.74,7215.60");
    }

}

