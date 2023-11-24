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

network DADCA_ACK_n20_u4_a8000m2_0
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
        @display("i=misc/sensor;p=1243.79,7881.59");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3014.47,7788.46");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3314.02,7972.68");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4931.79,7840.86");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5438.74,7866.99");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5982.81,7871.21");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=243.29,7700.32");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=722.89,7644.07");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1766.73,7688.47");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2427.82,7677.11");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3810.55,7537.40");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=319.32,7326.32");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1174.19,7388.57");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1528.13,7432.26");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2942.97,7426.97");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3336.91,7471.80");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4330.69,7459.99");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4878.38,7273.40");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5317.27,7496.40");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5668.98,7283.40");
    }

}

