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

network DADCA_ACK_n50_u3_a8000m2_7
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
        @display("i=misc/sensor;p=1010.01,7895.49");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1379.41,7888.96");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3325.04,7798.33");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3674.03,7910.90");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4042.62,7984.38");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5632.55,7788.97");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5976.48,7985.94");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=115.44,7610.28");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=566.60,7732.47");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=840.82,7509.00");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4445.81,7639.29");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4968.18,7718.67");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5958.98,7525.99");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1840.94,7316.78");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2295.23,7476.90");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2783.09,7347.79");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3352.85,7335.18");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3897.43,7469.85");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4705.52,7282.19");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5431.59,7475.68");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=534.65,7159.49");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=999.22,7134.46");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1392.10,7011.79");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4313.75,7199.02");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5039.42,7145.25");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5499.20,7086.55");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5923.96,7063.13");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2021.36,6865.91");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2462.89,6994.73");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3275.15,6914.39");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3725.07,6946.17");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=99.91,6531.96");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=582.02,6715.24");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=947.00,6541.85");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1612.33,6561.59");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2901.26,6690.30");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4890.03,6701.05");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5464.55,6557.37");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5996.84,6712.15");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1933.12,6296.05");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2466.02,6503.53");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3589.79,6370.40");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4046.99,6416.17");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4461.88,6426.23");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=60.53,6113.04");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=533.39,6154.27");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1057.24,6059.95");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2265.22,6081.63");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2942.93,6207.00");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5886.56,6210.09");
    }

}

