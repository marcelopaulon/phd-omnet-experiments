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

network DADCA_ACK_n50_u2_a8000m2_4
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
        @display("i=misc/sensor;p=701.48,7838.03");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1077.76,7862.83");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1767.87,7841.10");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3165.07,7794.63");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3719.44,7802.38");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5739.52,7765.75");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2401.48,7619.13");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4456.30,7551.14");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4875.46,7621.73");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=211.88,7449.14");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=682.26,7342.40");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1356.79,7388.18");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1955.31,7285.36");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2935.24,7498.24");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3400.01,7263.19");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4140.22,7380.06");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4762.22,7282.44");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5280.69,7490.82");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=970.65,7090.56");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2542.14,7039.30");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4404.17,7129.00");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5728.85,7127.89");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=369.26,6837.89");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1759.85,6950.94");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3956.86,6825.96");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5190.25,6878.17");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1409.91,6585.97");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2898.27,6742.87");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3259.42,6583.72");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4326.39,6633.75");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4756.74,6711.74");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5629.88,6759.28");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=55.69,6430.73");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=400.27,6308.86");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1016.84,6386.07");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2079.62,6458.08");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2613.24,6508.72");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3780.16,6311.95");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4822.12,6318.01");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5340.63,6310.99");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5769.20,6397.13");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1513.83,6105.70");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2908.21,6054.03");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5033.43,6031.41");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=160.87,6017.57");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=797.26,5984.43");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2513.57,5871.33");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3409.02,5854.77");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4371.88,5988.20");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5894.55,5803.23");
    }

}

