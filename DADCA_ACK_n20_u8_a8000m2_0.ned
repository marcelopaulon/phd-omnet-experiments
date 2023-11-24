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

network DADCA_ACK_n20_u8_a8000m2_0
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
        @display("i=misc/sensor;p=284.45,7954.97");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2534.06,7862.84");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3068.22,7841.74");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3556.62,7844.95");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4735.54,7989.76");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5316.51,7975.43");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5911.74,7771.36");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=634.90,7695.40");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1125.89,7650.55");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1537.84,7507.04");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2195.14,7698.41");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3440.23,7508.07");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=616.65,7343.73");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1044.08,7301.71");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2735.77,7327.04");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4097.34,7483.48");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4741.90,7311.96");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5355.41,7377.25");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5715.62,7481.42");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=7.14,7087.21");
    }

}

